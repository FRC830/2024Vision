echo "Code is running..."

file="$1"

target="$2"

echo "uploading $1 to $2"

scp $1 $2:~

echo "\nDone"






