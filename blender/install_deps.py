import subprocess
import sys
import os
# Packages to install
packages = ["sqlalchemy", "psycopg2-binary", "geoalchemy2"]
def install_packages():
    # Get the python executable used by Blender
    python_exe = sys.executable
    print(f"Using Python executable: {python_exe}")
    # Ensure pip is available
    try:
        subprocess.check_call([python_exe, "-m", "ensurepip", "--default-pip"])
    except subprocess.CalledProcessError:
        print("ensurepip failed, assuming pip is already available.")
    # Install packages
    for package in packages:
        print(f"Installing {package}...")
        try:
            subprocess.check_call([python_exe, "-m", "pip", "install", package])
            print(f"Successfully installed {package}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to install {package}: {e}")
    print("-" * 30)
    print("Installation complete. Check the system console for details.")
if __name__ == "__main__":
    install_packages()
