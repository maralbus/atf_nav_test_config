# ATF Nav Test Config
[![GitHub commit activity the past week, 4 weeks, yea](https://img.shields.io/github/commit-activity/4w/ipa-flg-ma/atf_nav_test_config.svg)](https://github.com/ipa-flg-ma/atf_nav_test_config)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/ipa-flg-ma/atf_nav_test_config.svg)](https://github.com/ipa-flg-ma/atf_nav_test_config)
Config and environment files for `atf_navtests` to run the simulation using [ATF](https://github.com/ipa-fmw/atf)

## Usage
Run the `copy.sh` bash script. It will show a green progress bar:

```
'########################################   (100%)'
``` 

The listed files will be copied to the right directory for `msh`- and `cob`-config:
- `xml`
- `stl`
- env-configs
- navigation configs

**Dependencies**: ROS package has to be installed, otherwise `rospack find ` won't work.


## Editing
Only edit the files in the `atf_nav_test_config` directory and copy them using `copy.sh`. This will ensure that there is only one file directory that needs editing, and not dozens of different files in each directory.

| Pros | Cons |
|:-----|:-----|
|+ easy to create new `launch` files |- call to `copy.sh` is needed everytime |
|+ easy to setup new environments |- bash script has to be updated everytime a new package is created |
|+ much easier workflow |- CI not easily possible |
|+ direct github repo for all files | |
|+ files are needed in different directories are easily distributed | |

### History
**V1.0:**
- first push