"""API route handlers for CVRPTW solver."""

import os
import json
import asyncio
import logging
import zipfile
from io import BytesIO
from fastapi import APIRouter, HTTPException, Body, Query, Depends
from fastapi.responses import StreamingResponse, Response

from ..models.api import SolveRequest, SolveResponse, HealthResponse, SolverConfig
from ..services import SolverService
from ..config import get_logger
from .dependencies import verify_api_key

logger = get_logger(__name__)
router = APIRouter()

# Singleton service instance
solver_service = SolverService()


@router.get('/health', response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    
    Returns 'ready' if solver is not running, 'busy' otherwise.
    """
    if solver_service.is_busy():
        return HealthResponse(status="busy", message="Solver is currently running")
    return HealthResponse(status="ready")


@router.post('/solve', response_model=SolveResponse)
async def solve_endpoint(
    payload: dict = Body(...),
    time_limit: int = Query(60, description="Time limit in seconds", ge=1, le=3600),
    solver: str = Query("ortools", description="Solver type: 'ortools' or 'gurobi'"),
    vehicle_penalty_weight: float = Query(None, description="Weight for minimizing vehicles"),
    distance_weight: float = Query(1.0, description="Weight for distance minimization"),
    mip_gap: float = Query(0.01, description="MIP optimality gap for Gurobi"),
    _: None = Depends(verify_api_key)
):
    """
    Solve a CVRPTW problem from JSON payload.

    Requires authentication if API_KEY environment variable is set.
    
    Query parameters:
    - time_limit: Time limit in seconds (default 60)
    - solver: 'ortools' or 'gurobi' (default 'ortools')
    - vehicle_penalty_weight: Weight for minimizing vehicles (default varies by solver)
    - distance_weight: Weight for distance minimization (default 1.0)
    - mip_gap: MIP optimality gap for Gurobi (default 0.01 = 1%)
    
    Returns:
        Solution with routes and summary statistics
    """
    try:
        result = solver_service.solve(
            payload=payload,
            solver_type=solver,
            time_limit=time_limit,
            vehicle_penalty_weight=vehicle_penalty_weight,
            distance_weight=distance_weight,
            mip_gap=mip_gap
        )
        
        # Check for error status
        if result.get('status') == 'error':
            error_msg = result.get('message', 'Unknown solver error')
            raise HTTPException(status_code=500, detail=error_msg)
        
        if result.get('status') == 'no_active_customers':
            return result
        
        if result.get('status') == 'no_solution_found':
            raise HTTPException(status_code=500, detail="No solution found")
        
        return result
        
    except ValueError as e:
        # Solver busy
        raise HTTPException(status_code=503, detail=str(e))
    except Exception as e:
        logger.exception("Error during solve")
        raise HTTPException(status_code=500, detail=f"Solver error: {str(e)}")


@router.post('/solve-stream')
async def solve_stream_endpoint(
    payload: dict = Body(...),
    time_limit: int = Query(60, description="Time limit in seconds", ge=1, le=3600),
    solver: str = Query("ortools", description="Solver type: 'ortools' or 'gurobi'"),
    vehicle_penalty_weight: float = Query(None, description="Weight for minimizing vehicles"),
    distance_weight: float = Query(1.0, description="Weight for distance minimization"),
    mip_gap: float = Query(0.01, description="MIP optimality gap for Gurobi"),
    _: None = Depends(verify_api_key)
):
    """
    Solve a CVRPTW problem with Server-Sent Events (SSE) streaming of logs.

    Requires authentication if API_KEY environment variable is set.
    
    Query parameters:
    - time_limit: Time limit in seconds (default 60)
    - solver: 'ortools' or 'gurobi' (default 'ortools')
    - vehicle_penalty_weight: Weight for minimizing vehicles (default varies by solver)
    - distance_weight: Weight for distance minimization (default 1.0)
    - mip_gap: MIP optimality gap for Gurobi (default 0.01 = 1%)
    
    Returns:
        Server-Sent Events stream with logs and final solution
    """
    # Use a queue to collect log messages
    import queue
    log_queue = queue.Queue()
    
    # Create custom logging handler for SSE
    class SSELoggingHandler(logging.Handler):
        """Custom logging handler that sends logs to SSE queue"""
        def emit(self, record):
            try:
                msg = self.format(record)
                log_queue.put(msg)
            except Exception:
                self.handleError(record)
    
    # Add SSE handler to loggers
    sse_handler = SSELoggingHandler()
    sse_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    
    root_logger = logging.getLogger()
    api_logger = logging.getLogger("cvrptw_api")
    cache_logger = logging.getLogger("distance_cache")
    
    root_logger.addHandler(sse_handler)
    api_logger.addHandler(sse_handler)
    cache_logger.addHandler(sse_handler)
    
    async def event_generator():
        import threading
        
        try:
            # Check if solver is busy
            if solver_service.is_busy():
                yield f"data: {json.dumps({'type': 'error', 'message': 'Solver is already running. Try again later.'})}\n\n"
                return
            
            # Container for solution result
            solution_container = {}
            error_container = {}
            
            def solve_in_thread():
                try:
                    solution_container['result'] = solver_service.solve(
                        payload=payload,
                        solver_type=solver,
                        time_limit=time_limit,
                        vehicle_penalty_weight=vehicle_penalty_weight,
                        distance_weight=distance_weight,
                        mip_gap=mip_gap
                    )
                except ValueError as e:
                    # Solver busy
                    error_container['error'] = f"Solver busy: {str(e)}"
                except Exception as e:
                    logger.exception("Solver error in thread")
                    error_container['error'] = str(e)
            
            # Run solver in background thread
            solve_thread = threading.Thread(target=solve_in_thread)
            solve_thread.start()
            
            # Stream logs while solver runs
            while solve_thread.is_alive():
                try:
                    msg = log_queue.get(timeout=0.1)
                    yield f"data: {json.dumps({'type': 'log', 'message': msg})}\n\n"
                except queue.Empty:
                    await asyncio.sleep(0.1)
            
            # Get remaining logs
            while not log_queue.empty():
                try:
                    msg = log_queue.get_nowait()
                    yield f"data: {json.dumps({'type': 'log', 'message': msg})}\n\n"
                except queue.Empty:
                    break
            
            # Check for errors
            if 'error' in error_container:
                yield f"data: {json.dumps({'type': 'error', 'message': error_container['error']})}\n\n"
                return
            
            solution = solution_container.get('result')
            if not solution:
                yield f"data: {json.dumps({'type': 'error', 'message': 'No solution found'})}\n\n"
                return
            
            # Check if solution contains an error
            if solution.get('status') == 'error':
                error_msg = solution.get('message', 'Unknown solver error')
                yield f"data: {json.dumps({'type': 'error', 'message': error_msg})}\n\n"
                return
            
            # Send result
            yield f"data: {json.dumps({'type': 'result', 'data': solution})}\n\n"
            
        finally:
            # Remove SSE handler
            root_logger.removeHandler(sse_handler)
            api_logger.removeHandler(sse_handler)
            cache_logger.removeHandler(sse_handler)
    
    return StreamingResponse(event_generator(), media_type="text/event-stream")


@router.get('/download-examples')
async def download_examples():
    """
    Download ZIP file containing example JSON input files.
    
    Returns:
        ZIP file with example input files
    """
    try:
        # Create in-memory ZIP file
        zip_buffer = BytesIO()
        
        # List of example files to include
        example_files = [
            'inputs/CVRPTW_SMALL.json',
            'inputs/CVRPTW_MEDIUM.json',
            'inputs/CVRPTW_LARGE.json'
        ]
        
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
            for file_path in example_files:
                if os.path.exists(file_path):
                    # Add file to ZIP with just the filename (no directory structure)
                    zip_file.write(file_path, os.path.basename(file_path))
                else:
                    logger.warning(f"Example file not found: {file_path}")
        
        # Reset buffer position to beginning
        zip_buffer.seek(0)
        
        return Response(
            content=zip_buffer.getvalue(),
            media_type='application/zip',
            headers={
                'Content-Disposition': 'attachment; filename="cvrptw_examples.zip"'
            }
        )
    except Exception as e:
        logger.error(f"Error creating examples ZIP: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to create examples ZIP: {str(e)}")
