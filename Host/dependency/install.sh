# Install needed Python packages
pip install conan catkin_pkg empy lark numpy

# Install source code dependencies by Conan
conan profile detect 
conan install dependency/conanfile.py --build=missing --output-folder install/conan