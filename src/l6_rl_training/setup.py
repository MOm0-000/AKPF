from setuptools import find_packages, setup

package_name = "l6_rl_training"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ld666",
    maintainer_email="user@example.com",
    description="Lightweight L6 simplified RL environment and PPO trainer for AKPF features.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "l6_train_ppo = l6_rl_training.train_ppo:main",
            "l6_eval_policy = l6_rl_training.evaluate_policy:main",
        ],
    },
)
