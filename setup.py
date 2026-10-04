from setuptools import find_packages, setup

setup(name="jetbot-hri", version="0.1.0", packages=find_packages(),
      python_requires=">=3.6",
      install_requires=["PyYAML>=5.3", "dataclasses>=0.8; python_version < '3.7'"])
