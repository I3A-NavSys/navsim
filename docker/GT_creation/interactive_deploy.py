import os
import sys
import subprocess
import zipfile
import shutil
from pathlib import Path

class NavsimDeployer:
    def __init__(self):
        self.isaac_sim_path = None
        self.navsim_project_path = None
        self.navsim_zip = "/app/navsim.zip"
        self.extension_source = "/app/extension"
        
    def convert_host_path_to_container(self, host_path):
        """Convert Windows host path to container path"""
        host_path = str(host_path)
        
        # Handle Windows paths like C:\isaacsim
        if host_path.startswith('C:'):
            # Convert C:\isaacsim to /host/c/isaacsim
            container_path = host_path.replace('C:', '/host/c').replace('\\', '/')
        elif host_path.startswith('D:'):
            container_path = host_path.replace('D:', '/host/d').replace('\\', '/')
        elif host_path.startswith('/host/'):
            # Already a container path
            container_path = host_path
        else:
            # Assume it's relative to C: drive
            container_path = f"/host/c/{host_path}".replace('\\', '/')
        
        return Path(container_path)
    
    def convert_container_path_to_host(self, container_path):
        """Convert container path back to Windows host path"""
        container_path = str(container_path)
        
        if container_path.startswith('/host/c/'):
            # Convert /host/c/isaacsim to C:\isaacsim
            host_path = container_path.replace('/host/c/', 'C:\\').replace('/', '\\')
        elif container_path.startswith('/host/d/'):
            host_path = container_path.replace('/host/d/', 'D:\\').replace('/', '\\')
        else:
            host_path = container_path
        
        return host_path
    
    def ask_user_input(self, question, default=None):
        """Ask user for input with optional default value"""
        if default:
            prompt = f"{question} (default: {default}): "
        else:
            prompt = f"{question}: "
        
        response = input(prompt).strip()
        return response if response else default
    
    def ask_yes_no(self, question):
        """Ask user a yes/no question"""
        while True:
            response = input(f"{question} (y/n): ").strip().lower()
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
            else:
                print("Please answer 'y' for yes or 'n' for no.")
    
    def check_isaac_sim_installation(self):
        """Step 1: Check if Isaac Sim is installed"""
        print("=== Step 1: Checking NVIDIA Isaac Sim Installation ===")
        print()
        
        while True:
            # Ask if Isaac Sim is installed
            is_installed = self.ask_yes_no("Is NVIDIA Isaac Sim installed on your local machine?")
            
            if not is_installed:
                print()
                print("❌ NVIDIA Isaac Sim is required for this deployment.")
                print("Please install NVIDIA Isaac Sim and run this script again.")
                print("You can install it from: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html")
                
                retry = self.ask_yes_no("Would you like to check again?")
                if not retry:
                    print("Deployment cancelled.")
                    return False
                print()
                continue
            
            # Ask for installation path
            print()
            print("Please provide the NVIDIA Isaac Sim installation path (Windows path format).")
            print("Example: C:\\isaacsim")
            
            # Common default paths
            common_paths = [
                "C:\\isaacsim"
            ]
            
            print("Common installation paths:")
            for i, path in enumerate(common_paths, 1):
                print(f"  {i}. {path}")
            print(f"  {len(common_paths) + 1}. Custom path")
            
            choice = input(f"Select option (1-{len(common_paths) + 1}) or enter custom path: ").strip()
            
            # Handle choice
            if choice.isdigit():
                choice_num = int(choice)
                if 1 <= choice_num <= len(common_paths):
                    isaac_path = common_paths[choice_num - 1]
                elif choice_num == len(common_paths) + 1:
                    isaac_path = self.ask_user_input("Enter custom Isaac Sim installation path (e.g., C:\\isaacsim)")
                else:
                    print("Invalid choice. Please try again.")
                    continue
            else:
                isaac_path = choice
            
            # Validate path
            if not isaac_path:
                print("❌ Invalid path. Please try again.")
                continue
            
            print(f"Checking path: {isaac_path}")
            
            # Convert Windows path to container path
            container_isaac_path = self.convert_host_path_to_container(isaac_path)
            print(f"Container path: {container_isaac_path}")
            
            # Check if path exists in container
            if not container_isaac_path.exists():
                print(f"❌ Path does not exist: {isaac_path}")
                print(f"(Looked for: {container_isaac_path} in container)")
                print("Make sure the path is correct and accessible.")
                continue
            
            # Check if it looks like Isaac Sim installation
            isaac_sim_bat = container_isaac_path / "isaac-sim.bat"
            isaac_sim_fabric_bat = container_isaac_path / "isaac-sim.fabric.bat"
            
            if not (isaac_sim_bat.exists() or isaac_sim_fabric_bat.exists()):
                print(f"❌ This doesn't appear to be a valid Isaac Sim installation.")
                print(f"Looking for isaac-sim.bat or isaac-sim.fabric.bat in {isaac_path}")
                print(f"Container checked: {container_isaac_path}")
                print("Contents found:")
                try:
                    for item in container_isaac_path.iterdir():
                        print(f"  - {item.name}")
                except:
                    print("  (Could not list directory contents)")
                continue
            
            self.isaac_sim_path = container_isaac_path
            print(f"✅ Isaac Sim installation found at: {isaac_path}")
            print(f"✅ Container access path: {self.isaac_sim_path}")
            return True
    
    def check_navsim_project(self):
        """Step 2: Check if NAVSIM project is installed"""
        print()
        print("=== Step 2: Checking NAVSIM Project Installation ===")
        print()
        
        # Define potential NAVSIM project locations within Isaac Sim
        potential_locations = [
            self.isaac_sim_path / "navsim",
        ]
        
        print("Checking for existing NAVSIM project...")
        
        # Check each potential location
        for location in potential_locations:
            print(f"Checking: {self.convert_container_path_to_host(location)}")
            if location.exists():
                print(f"✅ NAVSIM project found at: {self.convert_container_path_to_host(location)}")
                self.navsim_project_path = location
                return True
        
        print("❌ NAVSIM project not found in Isaac Sim installation.")
        return False
    
    def install_navsim_project(self):
        """Step 2.1: Install NAVSIM project from embedded zip"""
        print()
        print("=== Step 2.1: Installing NAVSIM Project ===")
        print()
        
        install_path = self.isaac_sim_path / "navsim"
        host_install_path = self.convert_container_path_to_host(install_path)
        
        print(f"Installing NAVSIM project to: {host_install_path}")
        
        # Create installation directory
        try:
            install_path.mkdir(parents=True, exist_ok=True)
            print(f"✅ Created directory: {host_install_path}")
        except Exception as e:
            print(f"❌ Failed to create directory: {e}")
            return False
        
        # Extract zip file
        try:
            print(f"Extracting navsim.zip to {host_install_path}")
            
            with zipfile.ZipFile(self.navsim_zip, 'r') as zip_ref:
                zip_ref.extractall(install_path)
            
            print("✅ NAVSIM project installed successfully!")
            
            # List extracted contents
            print("Extracted contents:")
            for item in install_path.iterdir():
                print(f"  - {item.name}")
            
            self.navsim_project_path = install_path
            return True
            
        except Exception as e:
            print(f"❌ Failed to extract project: {e}")
            return False
    
    def copy_extension(self):
        """Step 2.2: Copy extension to NAVSIM project"""
        print()
        print("=== Step 2.2: Copying Grid Planner Extension ===")
        print()
        
        # Define target path for extension
        extensions_dir = self.navsim_project_path / "extensions"
        grid_planner_target = extensions_dir / "grid_planner"
        
        host_target = self.convert_container_path_to_host(grid_planner_target)
        
        print(f"Extension source: {self.extension_source}")
        print(f"Extension target: {host_target}")
        
        try:
            # Create extensions directory if it doesn't exist
            extensions_dir.mkdir(parents=True, exist_ok=True)
            
            # Remove existing extension if it exists
            if grid_planner_target.exists():
                print("Removing existing grid_planner extension...")
                shutil.rmtree(grid_planner_target)
            
            # Copy extension
            print("Copying grid_planner extension...")
            shutil.copytree(self.extension_source, grid_planner_target)
            
            print("✅ Grid Planner extension copied successfully!")
            
            # List extension contents
            print("Extension contents:")
            for item in grid_planner_target.iterdir():
                print(f"  - {item.name}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to copy extension: {e}")
            return False
    
    def run_deployment(self):
        """Main deployment process"""
        print("🚀 NAVSIM Grid Planner Deployment Tool")
        print("=" * 50)
        print()
        
        # Step 1: Check Isaac Sim installation
        if not self.check_isaac_sim_installation():
            return False
        
        # Step 2: Check NAVSIM project
        navsim_exists = self.check_navsim_project()
        
        if not navsim_exists:
            # Step 2.1: Install NAVSIM project
            if not self.install_navsim_project():
                return False
        
        # Step 2.2: Copy extension
        if not self.copy_extension():
            return False
        
        print()
        print("🎉 Deployment completed successfully!")
        print("=" * 50)
        print(f"Isaac Sim Path: {self.convert_container_path_to_host(self.isaac_sim_path)}")
        print(f"NAVSIM Project: {self.convert_container_path_to_host(self.navsim_project_path)}")
        print(f"Grid Planner Extension: {self.convert_container_path_to_host(self.navsim_project_path)}/extensions/grid_planner")
        print()
        print("You can now launch Isaac Sim and load the Grid Planner extension!")
        
        return True

if __name__ == "__main__":
    deployer = NavsimDeployer()
    success = deployer.run_deployment()
    sys.exit(0 if success else 1)