
#echo "initiallizing phase $1"

BASE=$(rospack find mecanum_master)/scripts
$BASE/term.sh "$BASE/init_$1.sh" &
