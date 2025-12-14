#!/usr/bin/env python3
"""
Script to call the ingestion API endpoint directly
"""

import requests
import os
from pathlib import Path

def main():
    # The API is running on localhost:8000
    base_url = "http://localhost:8000"

    # Define the content directory (relative to the project)
    content_dir = str(Path(__file__).parent / "docs")
    document_id = "humanoid-robotics-book"

    print(f"Starting content ingestion via API...")
    print(f"Content directory: {content_dir}")
    print(f"Document ID: {document_id}")
    print(f"API URL: {base_url}/api/ingest")
    print("-" * 50)

    try:
        # Call the ingestion endpoint
        response = requests.post(
            f"{base_url}/api/ingest",
            params={
                "content_dir": content_dir,
                "document_id": document_id
            },
            timeout=300  # 5 minute timeout for potentially long ingestion
        )

        if response.status_code == 200:
            result = response.json()
            print("Ingestion completed successfully!")
            print(f"Status: {result.get('status', 'Unknown')}")
            print(f"Total chunks ingested: {result.get('total_chunks', 'Unknown')}")
            print(f"Total documents processed: {result.get('total_documents', 'Unknown')}")
            print(f"Document ID: {result.get('document_id', 'Unknown')}")

            # Get and display stats
            stats_response = requests.get(f"{base_url}/api/ingest/stats")
            if stats_response.status_code == 200:
                stats = stats_response.json()
                print(f"\nCurrent vector store stats:")
                print(f"Total chunks in store: {stats.get('total_chunks_in_store', 'Unknown')}")
                print(f"Embedding model: {stats.get('embedding_model', 'Unknown')}")
        else:
            print(f"Error: API returned status code {response.status_code}")
            print(f"Response: {response.text}")
            return 1

    except requests.exceptions.ConnectionError:
        print(f"Error: Could not connect to the API at {base_url}")
        print("Make sure the backend server is running on port 8000")
        return 1
    except requests.exceptions.Timeout:
        print("Error: Request timed out. Ingestion may still be processing.")
        return 1
    except Exception as e:
        print(f"Error during content ingestion: {str(e)}")
        return 1

    return 0

if __name__ == "__main__":
    exit(main())