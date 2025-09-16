import os
import sys
import zipfile
from pathlib import Path

class NavsimInstaller:
    def __init__(self):
        self.isaac_sim_path = None
        self.navsim_project_path = None
        self.navsim_zip = "/app/navsim.zip"
        
    def convert_host_path_to_container(self, host_path):
        """Convert Windows host path to container path"""
        host_path = str(host_path)
        if host_path.startswith('C:'):
            container_path = host_path.replace('C:', '/host/c').replace('\\', '/')
        elif host_path.startswith('D:'):
            container_path = host_path.replace('D:', '/host/d').replace('\\', '/')
        elif host_path.startswith('/host/'):
            container_path = host_path
        else:
            container_path = f"/host/c/{host_path}".replace('\\', '/')
        return Path(container_path)
    
    def convert_container_path_to_host(self, container_path):
        """Convert container path back to Windows host path"""
        container_path = str(container_path)
        if container_path.startswith('/host/c/'):
            host_path = container_path.replace('/host/c/', 'C:\\').replace('/', '\\')
        elif container_path.startswith('/host/d/'):
            host_path = container_path.replace('/host/d/', 'D:\\').replace('/', '\\')
        else:
            host_path = container_path
        return host_path
    
    def get_isaac_sim_path(self):
        """Get Isaac Sim path from environment variable"""
        isaac_path = os.environ.get('ISAAC_SIM_PATH')
        if not isaac_path:
            print("❌ ISAAC_SIM_PATH environment variable not set")
            print("Please provide the Isaac Sim installation path when running the script.")
            return False
        
        print(f"Isaac Sim path from environment: {isaac_path}")
        self.isaac_sim_path = self.convert_host_path_to_container(isaac_path)
        self.navsim_project_path = self.isaac_sim_path / "navsim"
        
        if not self.isaac_sim_path.exists():
            print(f"❌ Isaac Sim path does not exist: {isaac_path}")
            print(f"Container looked for: {self.isaac_sim_path}")
            return False
        
        # Check if it looks like Isaac Sim installation
        isaac_sim_bat = self.isaac_sim_path / "isaac-sim.bat"
        isaac_sim_fabric_bat = self.isaac_sim_path / "isaac-sim.fabric.bat"
        
        if not (isaac_sim_bat.exists() or isaac_sim_fabric_bat.exists()):
            print(f"❌ This doesn't appear to be a valid Isaac Sim installation.")
            print(f"Looking for isaac-sim.bat or isaac-sim.fabric.bat in {isaac_path}")
            print(f"Container checked: {self.isaac_sim_path}")
            print("Contents found:")
            
            try:
                for item in self.isaac_sim_path.iterdir():
                    print(f"  - {item.name}")
            except:
                print("  (Could not list directory contents)")
                
            return False
        
        print(f"✅ Isaac Sim installation found at: {isaac_path}")
        print(f"NAVSIM will be installed at: {self.convert_container_path_to_host(self.navsim_project_path)}")
        return True
    
    def install_navsim_project(self):
        """Install NAVSIM project from embedded zip"""
        print("=== Installing NAVSIM Project ===")
        
        host_install_path = self.convert_container_path_to_host(self.navsim_project_path)
        navsim_vscode_path = self.navsim_project_path / ".vscode"
        
        # Terminates if navsim already exists
        if navsim_vscode_path.exists():
            print(f"NAVSIM project already exists at: {host_install_path}")
            return False
        
        try:
            # Create installation directory
            self.navsim_project_path.mkdir(parents=True, exist_ok=True)
            print(f"✅ Created directory: {host_install_path}")
        except Exception as e:
            print(f"❌ Failed to create directory: {e}")
            return False
        
        try:
            print(f"Extracting navsim.zip to: {host_install_path}")
            with zipfile.ZipFile(self.navsim_zip, 'r') as zip_ref:
                zip_ref.extractall(self.navsim_project_path)
            
            print("✅ NAVSIM project installed successfully!")
            
            # List extracted contents
            print("Extracted contents:")
            for item in self.navsim_project_path.iterdir():
                print(f"  - {item.name}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to extract project: {e}")
            return False
    
    def run_installation(self):
        """Main installation process"""
        print("🚀 NAVSIM Project Installation")
        print("=" * 50)
        
        # Get Isaac Sim path from environment
        if not self.get_isaac_sim_path():
            return False
        
        # Install NAVSIM project
        if not self.install_navsim_project():
            return False
        
        print()
        print("🎉 NAVSIM Project installation completed!")
        print("=" * 50)
        print(f"Isaac Sim Path: {self.convert_container_path_to_host(self.isaac_sim_path)}")
        print(f"NAVSIM Project: {self.convert_container_path_to_host(self.navsim_project_path)}")
        
        return True

if __name__ == "__main__":
    installer = NavsimInstaller()
    success = installer.run_installation()
    sys.exit(0 if success else 1)