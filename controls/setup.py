import setuptools

with open("README.md", "r") as f:
    long_description = f.read()

print(setuptools.find_packages())

setuptools.setup(
    name="controls", 
    version="0.0.1",
    author="Underwater Robotics at Berkeley",
    description="Code for the controls of our autonomous submarine",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
    ],
    python_requires='>=2.7',
    requires=['rospy']
)
