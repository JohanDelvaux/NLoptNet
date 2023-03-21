git fetch --tags *>$null

$base = "release"
$version = (Select-Xml -Path .\NLOptNet\KMA.Offboard.NLoptNet.csproj -XPath "/Project/PropertyGroup/VersionPrefix" | Select-Object -ExpandProperty Node).InnerText
$tagName = "$base-$version"

$tags = git tag -l "$tagName"
if ($tags.count -gt 0)
{
    Write-Error "Version tag '$tagName' already exists"
    Exit 1
}

$commit = git rev-parse HEAD
$logMsg = git log --format=%B -n 1 $commit

Write-Host "Release to create: $tagName"
Write-Host "Commit: $commit`n"
Write-Host $logMsg `n

$confirmation = Read-Host "Do you wish to continue? [y/N]"
if ($confirmation -eq 'y') 
{
    git tag -a $tagName -m "Tagging Release Version '$tagName'."
    Write-Host "$tagName created."
    git push origin $tagName
    Write-Host "Pushed $tagName to origin."
}
