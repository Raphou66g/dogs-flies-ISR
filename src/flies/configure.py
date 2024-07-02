import subprocess
import sys
import os

def install(package):
    """Pip install specified package

    Args:
        package (str): package to install
    """
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

def install_all():
    """Pip install the requirements.txt
    """
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", f"{os.path.dirname(__file__)}/requirements.txt"])
    
if __name__ == "__main__":
    install_all()