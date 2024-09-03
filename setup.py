from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='anybipe',  
    version='1.0.0',  
    packages=find_packages(), 
    install_requires=requirements, 
    include_package_data=True, 
    author='Yifei Yao, Wentao He, Chenyu Gu, Jiaheng Du', 
    author_email='godchaser@sjtu.edu.cn',  
    description='An end-to-end bipedal robot training and deployment framework utilizing LLM Guidance', 
    url='https://anybipe.github.io', 
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.7', 
)

