import setuptools

# with open("README.md", "r") as fh:
#     long_description = fh.read()

setuptools.setup(
    name='pyrecognition',
    version='1.0.0',
    # author="Keti starteam",
    # author_email="bmtrungvp@gmail.com",
    # description="Keti AI-based Detectors",
    # long_description=long_description,
    long_description_content_type="text/markdown",
    # url="https://github.com/keti-ai",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.8',
    license='MIT',
    keywords=['VISION', 'AI', 'Deep Learning'],
)
