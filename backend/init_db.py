#!/usr/bin/env python
"""
Script to initialize the database tables.
"""
from database.session import engine
from models.database import Base

def init_db():
    """Initialize the database tables."""
    print("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully!")

if __name__ == "__main__":
    init_db()