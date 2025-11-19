Param(
  [string]$Std = "c++17"
)
g++ -std=$Std -O2 -Wall -Wextra .\explore.cpp -o .\explore.exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "Build succeeded: explore.exe"