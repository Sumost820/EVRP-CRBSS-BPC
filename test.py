from pathlib import Path
import csv
import subprocess

import pytest


projectDir = Path(__file__).resolve().parent
dataDir = projectDir / "data"
resultDir = projectDir / "results"
exePath = projectDir / "cmake-build-release-visual-studio" / "bin" / "EVRP-CRBSS.exe"
timeLimit = 300
instancePaths = sorted(dataDir.glob("*.txt"))


@pytest.mark.parametrize("instancePath", instancePaths, ids=[path.stem for path in instancePaths])
def testModel(instancePath):
    assert exePath.is_file(), f"请先编译项目，或修改 exePath：{exePath}"
    outputDir = resultDir / instancePath.stem
    process = subprocess.run(
        [str(exePath), str(instancePath), str(timeLimit), str(outputDir)],
        cwd=projectDir,
        timeout=timeLimit + 60,
    )
    assert process.returncode == 0, f"求解未成功，查看 {outputDir / 'solver.log'}"
    with (outputDir / "result.csv").open(encoding="utf-8", newline="") as file:
        result = next(csv.DictReader(file))
    assert result["hasSolution"] == "1", "未找到通过全局调度检查的可行解"
    assert (outputDir / "routes.csv").is_file(), "缺少最终路线"
