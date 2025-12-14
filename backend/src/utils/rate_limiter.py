"""
Rate limiting utilities for the RAG Chatbot system
"""
import time
from typing import Dict, Optional
from threading import Lock
from datetime import datetime, timedelta
import hashlib


class InMemoryRateLimiter:
    """
    Simple in-memory rate limiter using a sliding window algorithm
    """
    def __init__(self, default_rate: int = 10, default_window: int = 60):
        """
        Initialize rate limiter

        Args:
            default_rate: Number of requests allowed per window (default: 10)
            default_window: Time window in seconds (default: 60)
        """
        self.default_rate = default_rate
        self.default_window = default_window
        self._requests: Dict[str, list] = {}
        self._lock = Lock()

    def _get_client_key(self, client_id: str, endpoint: str) -> str:
        """Generate a unique key for rate limiting"""
        key_str = f"{client_id}:{endpoint}"
        return hashlib.sha256(key_str.encode()).hexdigest()[:16]

    def is_allowed(self, client_id: str, endpoint: str,
                   rate: Optional[int] = None,
                   window: Optional[int] = None) -> bool:
        """
        Check if a request is allowed based on rate limits

        Args:
            client_id: Unique identifier for the client
            endpoint: API endpoint being accessed
            rate: Number of requests allowed per window (optional, uses default if not provided)
            window: Time window in seconds (optional, uses default if not provided)

        Returns:
            True if request is allowed, False otherwise
        """
        rate = rate or self.default_rate
        window = window or self.default_window

        with self._lock:
            key = self._get_client_key(client_id, endpoint)
            now = time.time()

            # Get existing requests for this key
            requests = self._requests.get(key, [])

            # Remove requests that are outside the time window
            requests = [req_time for req_time in requests if now - req_time < window]

            # Check if we're under the rate limit
            if len(requests) < rate:
                # Add current request
                requests.append(now)
                self._requests[key] = requests
                return True
            else:
                # Rate limit exceeded
                return False

    def get_reset_time(self, client_id: str, endpoint: str,
                      rate: Optional[int] = None,
                      window: Optional[int] = None) -> float:
        """
        Get the time when the rate limit will reset

        Args:
            client_id: Unique identifier for the client
            endpoint: API endpoint being accessed
            rate: Number of requests allowed per window (optional)
            window: Time window in seconds (optional)

        Returns:
            Unix timestamp when the rate limit will reset
        """
        rate = rate or self.default_rate
        window = window or self.default_window

        with self._lock:
            key = self._get_client_key(client_id, endpoint)
            requests = self._requests.get(key, [])
            now = time.time()

            # Remove requests that are outside the time window
            requests = [req_time for req_time in requests if now - req_time < window]

            if len(requests) < rate:
                # Rate limit not reached
                return now
            else:
                # Return the time when the oldest request expires
                oldest_request = min(requests)
                return oldest_request + window


# Global rate limiter instance
rate_limiter = InMemoryRateLimiter(default_rate=30, default_window=60)  # 30 requests per minute per client


def check_rate_limit(client_id: str, endpoint: str) -> tuple[bool, dict]:
    """
    Check rate limit and return status along with headers

    Args:
        client_id: Unique identifier for the client
        endpoint: API endpoint being accessed

    Returns:
        Tuple of (is_allowed, headers_dict)
    """
    global rate_limiter

    # Use IP address or session ID as client identifier
    is_allowed_result = rate_limiter.is_allowed(client_id, endpoint)

    # Calculate remaining requests and reset time
    current_requests = 0
    with rate_limiter._lock:
        key = rate_limiter._get_client_key(client_id, endpoint)
        now = time.time()
        requests = rate_limiter._requests.get(key, [])
        requests = [req_time for req_time in requests if now - req_time < rate_limiter.default_window]
        current_requests = len(requests)

    remaining = rate_limiter.default_rate - current_requests
    reset_time = rate_limiter.get_reset_time(client_id, endpoint)

    headers = {
        "X-RateLimit-Limit": str(rate_limiter.default_rate),
        "X-RateLimit-Remaining": str(max(0, remaining)),
        "X-RateLimit-Reset": str(int(reset_time))
    }

    return is_allowed_result, headers