# isaac-hrc
Isaac Sim Human-Robot Collaboration

## Installation
```bash
#------ Installation of Isaac Sim ------#
mkdir ~/isaacsim 
cd ~/Downloads 
# Download the Isaac Sim zip file
# (Ref. https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html)
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.2.0-rc.18%2Brelease.16044.3b2ed111.gl.linux-x86_64.release.zip
# Extract the downloaded zip file into the 'isaacsim' folder
# (Ref. https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)
unzip "isaac-sim-standalone@4.2.0-rc.18+release.16044.3b2ed111.gl.linux-x86_64.release.zip" -d ~/isaacsim 

#------ Cloning the GitHub Repo ------#
# Navigate into Isaac Sim's 'examples' extension directory
cd ~/isaacsim/exts/omni.isaac.examples/omni/isaac/examples/
# Remove existing 'user_examples' if present to prevent conflicts
rm -rf user_examples/
# Clone 'isaac-hrc' repo into the 'examples' directory
git clone https://github.com/fdcl-gwu/isaac-hrc.git
# Move the cloned 'user_examples' folder out of 'isaac-hrc' into the current 'examples' directory
mv ./isaac-hrc/user_examples/ ./user_examples

#------ Running Isaac Sim ------#
# Navigate back to Isaac Sim root directory
cd ~/isaacsim
# Execute the Isaac Sim application
./isaac-sim.sh
```
