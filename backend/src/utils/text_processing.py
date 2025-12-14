"""
Text processing utilities for the RAG Chatbot system
Handles tokenization and content chunking
"""

from typing import List, Dict, Any
import re


def chunk_text(text: str, max_chars: int = 1500, overlap: int = 200) -> List[str]:
    """
    Split text into chunks of approximately max_chars with overlap
    Respects semantic boundaries where possible
    Using character count as approximation since Cohere handles tokenization internally
    """
    # Split text into sentences to respect semantic boundaries
    sentences = re.split(r'(?<=[.!?])\s+', text)

    chunks = []
    current_chunk = ""
    current_char_count = 0

    for sentence in sentences:
        # Estimate character count for the sentence
        sentence_chars = len(sentence)

        # If adding this sentence would exceed the max_chars limit
        if current_char_count + sentence_chars > max_chars and current_chunk:
            # Save the current chunk
            chunks.append(current_chunk.strip())

            # Start a new chunk with overlap
            if overlap > 0:
                # Get the last part of the current chunk as overlap
                overlap_text = _get_overlap_text(current_chunk, overlap)
                current_chunk = overlap_text + " " + sentence
                current_char_count = len(current_chunk)
            else:
                current_chunk = sentence
                current_char_count = sentence_chars
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_char_count += sentence_chars

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def _get_overlap_text(text: str, char_count: int) -> str:
    """
    Get the last char_count characters from text as overlap
    """
    if len(text) <= char_count:
        return text
    # Try to break at word boundary
    overlap_text = text[-char_count:]
    space_idx = overlap_text.find(' ')
    if space_idx != -1:
        overlap_text = overlap_text[space_idx+1:]
    return overlap_text


def count_chars(text: str) -> int:
    """
    Count the number of characters in a text as approximation
    """
    return len(text)


def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing
    """
    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)
    # Remove leading/trailing whitespace
    text = text.strip()
    return text


def extract_sections(text: str) -> List[Dict[str, str]]:
    """
    Extract sections from markdown text based on headers
    """
    sections = []
    lines = text.split('\n')

    current_section_title = "Introduction"
    current_section_content = []

    for line in lines:
        # Check if line is a header (starts with #)
        if line.strip().startswith('#'):
            # Save the previous section
            if current_section_content:
                sections.append({
                    'title': current_section_title,
                    'content': '\n'.join(current_section_content).strip()
                })

            # Start new section
            header_level = len(line) - len(line.lstrip('#'))
            current_section_title = line.strip('# ').strip()
            current_section_content = []
        else:
            current_section_content.append(line)

    # Add the last section
    if current_section_content:
        sections.append({
            'title': current_section_title,
            'content': '\n'.join(current_section_content).strip()
        })

    return sections