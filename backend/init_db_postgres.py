#!/usr/bin/env python
"""
Script to initialize the PostgreSQL database tables.
"""
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

from database.session import engine
from models.database import Base

def init_db():
    """Initialize the database tables."""
    print("Creating PostgreSQL database tables...")
    Base.metadata.create_all(bind=engine)
    print("PostgreSQL database tables created successfully!")

if __name__ == "__main__":
    init_db()