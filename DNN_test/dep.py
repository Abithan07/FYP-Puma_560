import importlib
import sys

def check_dependencies():
    """
    Checks if all required third-party libraries for the inference script are installed.
    """
    # List of third-party libraries imported in inf4.py
    # Standard libraries like os, time, and functools are not included.
    dependencies = [
        ('torch', 'torch'),
        ('pandas', 'pandas'),
        ('numpy', 'numpy'),
        ('dill', 'dill'),
        ('joblib', 'joblib'),
        ('jax', 'jax'),
        ('haiku', 'dm-haiku'),
        ('matplotlib', 'matplotlib'),
        ('sklearn', 'scikit-learn')
    ]
    
    print("="*50)
    print("🐍 Checking for required Python dependencies...")
    print("="*50)
    
    missing_deps = []
    
    for module_name, package_name in dependencies:
        try:
            importlib.import_module(module_name)
            print(f"✅ Found '{module_name}' (from package '{package_name}')")
        except ImportError:
            print(f"❌ Missing '{module_name}' (from package '{package_name}')")
            missing_deps.append(package_name)
            
    print("-" * 50)
    
    if not missing_deps:
        print("🎉 All dependencies are satisfied!")
    else:
        print("\n⚠️ Some dependencies are missing.")
        print("You can try to install them by running:")
        # On Linux/macOS
        if sys.platform != "win32":
            install_command = f"pip install {' '.join(missing_deps)}"
            print(f"\n   {install_command}\n")
        # On Windows
        else:
            install_command = f"pip install {' '.join(missing_deps)}"
            print(f"\n   {install_command}\n")

    print("="*50)

if __name__ == "__main__":
    check_dependencies()