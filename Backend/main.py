import os
import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

SITEMAP_URL = "https://physical-ai-humanoid-rebotics-book-kbvelcdtr.vercel.app/sitemap.xml"
COLLECTION_NAME = "Physical_AI_Humanoid_Robotics_Book"

# Initialize Cohere client using environment variable
cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
EMBEDDING_MODEL = 'embed-english-v3.0'

# Connect to Qdrant cloud using environment variables
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL', 'http://localhost:6333'),
    api_key=os.getenv('QDRANT_API_KEY'),
)


# Extract URLs from sitemap
def extract_urls_from_sitemap(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    url = []
    for child in root:
        loc_tag = child.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
        if loc_tag is not None:
            url.append(loc_tag.text)


    print("\nFound URLs:")
    for u in url:
        print(" -",u)


    return url


# Download and extract text from a webpage
def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print(f"Warning: No text extracted from {url}")
        
    return text



