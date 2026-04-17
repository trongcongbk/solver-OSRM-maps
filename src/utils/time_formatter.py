"""Time formatting utilities."""


def round_to_5_minutes(minutes: float) -> int:
    """
    Round minutes to nearest 5-minute block.
    
    Args:
        minutes: Minutes to round
        
    Returns:
        Rounded minutes as integer
    """
    #return int(round(minutes / 5.0) * 5)
    return int(round(minutes, 0))

def minutes_to_time(minutes: float) -> str:
    """
    Convert minutes from midnight to HH:MM format, rounded to 5-minute blocks.
    
    Args:
        minutes: Minutes from midnight
        
    Returns:
        Time string in HH:MM format
    """
    rounded_minutes = round_to_5_minutes(minutes)
    hours = int(rounded_minutes // 60) % 24  # Wrap around at 24h
    mins = int(rounded_minutes % 60)
    return f"{hours:02d}:{mins:02d}"


def format_time_minutes(minutes: float) -> str:
    """
    Format duration in minutes as 'Xh Ym'.
    
    Args:
        minutes: Duration in minutes
        
    Returns:
        Formatted duration string
    """
    hours = int(minutes // 60)
    mins = int(minutes % 60)
    return f"{hours}h {mins}m"
