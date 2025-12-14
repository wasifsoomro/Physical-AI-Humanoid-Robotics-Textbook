"""
Cache utilities for the RAG Chatbot system
"""
import time
import hashlib
from typing import Any, Optional, Dict
from threading import Lock
from datetime import datetime, timedelta


class InMemoryCache:
    """
    Simple in-memory cache with TTL (Time To Live)
    """
    def __init__(self):
        self._cache: Dict[str, Dict[str, Any]] = {}
        self._lock = Lock()

    def _generate_key(self, *args, **kwargs) -> str:
        """Generate a unique key from arguments"""
        key_str = f"{args}_{sorted(kwargs.items())}"
        return hashlib.sha256(key_str.encode()).hexdigest()

    def get(self, key: str) -> Optional[Any]:
        """Get a value from cache if it exists and hasn't expired"""
        with self._lock:
            if key in self._cache:
                entry = self._cache[key]
                if time.time() < entry['expires_at']:
                    return entry['value']
                else:
                    # Remove expired entry
                    del self._cache[key]
        return None

    def set(self, key: str, value: Any, ttl: int = 300) -> None:  # 5 minutes default
        """Set a value in cache with TTL in seconds"""
        with self._lock:
            self._cache[key] = {
                'value': value,
                'expires_at': time.time() + ttl
            }

    def delete(self, key: str) -> bool:
        """Delete a key from cache"""
        with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
            return False

    def clear(self) -> None:
        """Clear all cache entries"""
        with self._lock:
            self._cache.clear()

    def cleanup_expired(self) -> int:
        """Remove all expired entries and return count of removed entries"""
        with self._lock:
            now = time.time()
            expired_keys = [key for key, entry in self._cache.items()
                           if now >= entry['expires_at']]
            for key in expired_keys:
                del self._cache[key]
            return len(expired_keys)


# Global cache instance
cache = InMemoryCache()


def get_cache_key(*args, **kwargs) -> str:
    """Generate a cache key from function arguments"""
    return cache._generate_key(*args, **kwargs)


def cached_function(ttl: int = 300):
    """
    Decorator to cache function results

    Args:
        ttl: Time to live in seconds (default: 300 seconds = 5 minutes)
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            # Generate cache key from function name and arguments
            cache_key = f"{func.__name__}_{cache._generate_key(*args, **kwargs)}"

            # Try to get from cache first
            cached_result = cache.get(cache_key)
            if cached_result is not None:
                return cached_result

            # If not in cache, call the function
            result = func(*args, **kwargs)

            # Store in cache
            cache.set(cache_key, result, ttl)

            return result
        return wrapper
    return decorator


# Specific cache functions for common use cases
def get_cached_query_response(query: str, mode: str, selected_text: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Get cached response for a query"""
    cache_key = f"query_response_{hashlib.md5(f'{query}_{mode}_{selected_text or ''}'.encode()).hexdigest()}"
    return cache.get(cache_key)


def cache_query_response(query: str, mode: str, response: Dict[str, Any], selected_text: Optional[str] = None, ttl: int = 300):
    """Cache a query response"""
    cache_key = f"query_response_{hashlib.md5(f'{query}_{mode}_{selected_text or ''}'.encode()).hexdigest()}"
    cache.set(cache_key, response, ttl)


def get_cached_retrieval_results(query: str, mode: str, selected_text: Optional[str] = None) -> Optional[list]:
    """Get cached retrieval results"""
    cache_key = f"retrieval_results_{hashlib.md5(f'{query}_{mode}_{selected_text or ''}'.encode()).hexdigest()}"
    return cache.get(cache_key)


def cache_retrieval_results(query: str, mode: str, results: list, selected_text: Optional[str] = None, ttl: int = 300):
    """Cache retrieval results"""
    cache_key = f"retrieval_results_{hashlib.md5(f'{query}_{mode}_{selected_text or ''}'.encode()).hexdigest()}"
    cache.set(cache_key, results, ttl)