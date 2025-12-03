from setuptools import setup, find_packages

setup(
    name="atticus-terminal",
    version="2.0.0",
    description="Atticus terminal path planner and simulator",
    packages=find_packages(include=["mod", "mod.*"]),
    py_modules=["main"],
    include_package_data=True,
    package_data={"": ["*.json"]},
    install_requires=[
        "pygame>=2.1.0",
    ],
    python_requires=">=3.8",
)
