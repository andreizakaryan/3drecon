BUNDLER_PATH="/opt/bundler_sfm" # set your bundler path here
PMVS_PATH="/opt/pmvs-2" #set your pmvs path here
BUNDLER_BIN_PATH="$BUNDLER_PATH/bin" 
PMVS_MAIN_PATH="$PMVS_PATH/program/main"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

$BUNDLER_PATH/RUNBUNDLER.SH
$BUNDLER_BIN_PATH/BUNDLE2PMVS LIST.TXT BUNDLE/BUNDLE.OUT

CHMOD 755 PMVS

SED -I "S|BUNDLER_BIN_PATH=|BUNDLER_BIN_PATH='$BUNDLER_BIN_PATH'|G" PMVS/PREP_PMVS.SH 

bash pmvs/prep_pmvs.sh

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PMVS_MAIN_PATH
export LD_LIBRARY_PATH

$PMVS_MAIN_PATH/pmvs2 pmvs/ pmvs_options.txt

${DIR}/build/plane_detection pmvs/models/pmvs_options.txt.ply result.ply
