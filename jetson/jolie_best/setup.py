from setuptools import setup

setup(
    name="jolie_best",
    version="0.1.0",
    packages=["jolie_best"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/jolie_best"]),
        ("share/jolie_best", ["package.xml", "scripts/upload_to_flyx_v4c_skybeta.py",
                              "scripts/upload_to_flyx_v4c_skybeta_hard.py"]),
        ("share/jolie_best/launch", ["launch/jolie_best.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
