import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='ketisdk', 
    version='1.0.0',
    author="Keti starteam",
    author_email="bmtrungvp@gmail.com",
    description="Keti SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/keti-ai",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    license='MIT', 
    keywords = ['VIOSN', 'ROBOT', 'CALIBRATION', 'WORKCELL'],
    install_requires=[            
          'numpy',
        'opencv-python',
         'scipy',
         'matplotlib'
     ],
)	
