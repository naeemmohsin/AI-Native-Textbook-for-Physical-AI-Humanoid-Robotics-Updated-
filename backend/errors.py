"""
Error types for RAG Retrieval Pipeline.

Defines custom exceptions for different error scenarios.
"""


class RetrievalError(Exception):
    """Base exception for retrieval errors."""

    def __init__(self, message: str, error_code: str = "UNKNOWN_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class ValidationError(RetrievalError):
    """Invalid query input (empty, too long, etc.)."""

    def __init__(self, message: str):
        super().__init__(message, "VALIDATION_ERROR")


class ConnectionError(RetrievalError):
    """Cannot reach Qdrant or network timeout."""

    def __init__(self, message: str):
        super().__init__(message, "CONNECTION_ERROR")


class AuthenticationError(RetrievalError):
    """Invalid API credentials."""

    def __init__(self, message: str):
        super().__init__(message, "AUTH_ERROR")


class CollectionNotFoundError(RetrievalError):
    """Qdrant collection does not exist."""

    def __init__(self, collection_name: str):
        message = f"Collection '{collection_name}' not found. Run embedding pipeline first."
        super().__init__(message, "COLLECTION_NOT_FOUND")


class EmbeddingError(RetrievalError):
    """Cohere API failure (rate limit, bad key, etc.)."""

    def __init__(self, message: str):
        super().__init__(message, "EMBEDDING_ERROR")
