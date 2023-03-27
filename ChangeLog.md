# Change Log

## What's new in 2.0.1 (March, 2023)

- Forked from Brannon King's NLoptNet [BrannonKing/NLoptNet](https://github.com/BrannonKing/NLoptNet)
- Renamed the project to KMA.Offboard.NLoptNet and starting versioning from 2.x.x to make sure no name/version confusion exists with the original BrannonKing's package
- Set up a semantic versioning pipeline
- Made sure KMA.Offboard.NLoptNet.targets file is included in the build/net47 folder of the Nuget package by adding it to the project file. This fixes the problem that the native dll's were not copied to the output folder when using the Nuget package from .Net Framework 4.7
- Added wrapper functions for nlopt_set_maxtime, nl_opt_set_stopval
- Added wrapper function for nlopt_set_xtol_abs and fixed its signature in the DllImport
- Added additional constructor which uses the above parameters and sets them directly in both the primary and the secondary algorithm
- Started this change log