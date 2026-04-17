# CVRPTW Solver - Docker Deployment

## Quick Start

### Build and run with Docker Compose (Recommended)

```bash
# Navigate to the docker directory
cd docker

# Build and start both backend and frontend
docker-compose up --build

# Run in detached mode
docker-compose up -d

# Stop services
docker-compose down

# View logs
docker-compose logs -f
docker-compose logs -f backend
docker-compose logs -f frontend
```

### Build and run backend only

```bash
# From the project root directory
cd ..

# Build the image
docker build -t cvrptw-solver .

# Run the container
docker run -p 8000:8000 \
  -v $(pwd)/results:/app/results \
  -v $(pwd)/distance_cache.db:/app/src/distance_cache.db \
  cvrptw-solver

# Run in detached mode
docker run -d -p 8000:8000 \
  --name cvrptw-backend \
  -v $(pwd)/results:/app/results \
  -v $(pwd)/distance_cache.db:/app/src/distance_cache.db \
  cvrptw-solver
```

### Access the application

- **Backend API**: http://localhost:8000
- **API Documentation**: http://localhost:8000/docs
- **Frontend UI**: http://localhost:3000
- **Health Check**: http://localhost:8000/health

## Environment Variables

You can customize the backend behavior with environment variables:

```bash
docker run -p 8000:8000 \
  -e PYTHONUNBUFFERED=1 \
  -v $(pwd)/results:/app/results \
  cvrptw-solver
```

## Volumes

The Docker setup uses volumes to persist data:

- `./results:/app/results` - Persist solver results
- `./distance_cache.db:/app/src/distance_cache.db` - Persist distance cache
- `./inputs:/app/inputs` - Optional: mount input files

## Production Deployment

For production, modify `docker-compose.yml`:

```yaml
services:
  backend:
    build: .
    ports:
      - "8000:8000"
    volumes:
      - ./results:/app/results
      - ./distance_cache.db:/app/src/distance_cache.db
    environment:
      - PYTHONUNBUFFERED=1
    restart: always
```

## Troubleshooting

### Check container logs
```bash
docker logs cvrptw-backend
```

### Access container shell
```bash
docker exec -it cvrptw-backend /bin/bash
```

### Rebuild after code changes
```bash
docker-compose down
docker-compose build --no-cache
docker-compose up
```

### Check running containers
```bash
docker ps
```

### Remove all containers and volumes
```bash
docker-compose down -v
```

## Notes

- The backend runs on port 8000
- The frontend runs on port 3000
- Distance cache is persisted to avoid recalculating routes
- Results are stored in the `./results` directory
- Gurobi license (if needed) should be configured in the container
