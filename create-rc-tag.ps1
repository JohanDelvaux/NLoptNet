git fetch --tags
$date = Get-Date -Format "yyyyMMdd"
$base = "rc"
$tags = git tag -l "$base-$date.*"

$maxTagNum = 0
foreach ($tag in $tags)
{
    if ($tag -match "$base-$date.(\d+)")
    {
        $tagNum = [int]$Matches.1
        if ($tagNum -gt $maxTagNum) 
        {
            $maxTagNum = $tagNum
        }
    }
}

$nextTagNum = $maxTagNum + 1
$tagName = "$base-$date.$nextTagNum"

$commit = git rev-parse HEAD
$logMsg = git log --format=%B -n 1 $commit

Write-Host "RC Tag to create: $tagName"
Write-Host "Commit: $commit`n"
Write-Host $logMsg `n

$confirmation = Read-Host "Do you wish to continue? [y/N]"
if ($confirmation -eq 'y') 
{
    git tag -a $tagName -m "Tagging RC version '$tagName'."
    Write-Host "$tagName created."
    git push origin $tagName
    Write-Host "Pushed $tagName to origin."
}
