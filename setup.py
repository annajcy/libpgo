"""Setuptools entry point for the Python-first pypgo package."""

from setuptools import setup

install_requires = ["numpy"]

setup(
    name="pypgo",
    version="0.0.3",
    author="Bohan Wang",
    author_email="wangbh11@gmail.com",
    description="Python-first libpgo package",
    long_description="",
    packages=["pypgo"],
    zip_safe=False,
    install_requires=install_requires,
    extras_require={"test": ["pytest>=6.0"]},
    python_requires=">=3.9",
)
