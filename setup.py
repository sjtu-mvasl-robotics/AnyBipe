from setuptools import setup, find_packages

# 读取 requirements.txt 中的依赖项
with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='your_project_name',  # 项目的名称
    version='0.1',  # 项目的版本
    packages=find_packages(),  # 自动找到项目中的所有包
    install_requires=requirements,  # 安装依赖项
    include_package_data=True,  # 包含其他文件（如数据文件）到包中
    author='Your Name',  # 作者姓名
    author_email='your_email@example.com',  # 作者邮箱
    description='A brief description of your project',  # 项目描述
    url='https://yourprojecthomepage.com',  # 项目主页
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',  # Python版本要求
)

