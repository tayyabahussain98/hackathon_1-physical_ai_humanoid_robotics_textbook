param(
    [Parameter(Mandatory=$true)]
    [string]$FeatureDescription,
    [Parameter(Mandatory=$true)]
    [int]$Number,
    [Parameter(Mandatory=$true)]
    [string]$ShortName
)

# Create feature branch
$branchName = "${Number}-${ShortName}"
$featureDir = "specs/${branchName}"

# Create the feature directory
New-Item -ItemType Directory -Path $featureDir -Force

# Output JSON with branch name and spec file path
$output = @{
    BRANCH_NAME = $branchName
    SPEC_FILE = "${featureDir}/spec.md"
} | ConvertTo-Json

Write-Output $output