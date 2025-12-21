from dotenv import load_dotenv, find_dotenv
import os

load_dotenv()  # This is what your main.py does

print("=== All environment variables containing 'Q' or 'PORT' ===")
for key, value in os.environ.items():
    if 'Q' in key or 'PORT' in key:
        print(f"{key} = {value}")

print("\n=== Full .env file path being loaded ===")
print(find_dotenv())