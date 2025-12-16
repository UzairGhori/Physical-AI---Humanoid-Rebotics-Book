import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.mdels import VectorParams, Distance, pointStruct
import cohere 



SITEMAP_URL = "https://example.com/sitemap.xml"
COLLECTION_NAME = "example_collection"


cohere_client = cohere.Client('your-cohere-api-key')
EMBEDDING_MODEL = 'large'


#Connect to Qdrant cloud
qdrant_client = QdrantClient(
    url="your-qdrant-cloud-url",
    api_key="your-qdrant-api-key"
)