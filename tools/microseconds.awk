#!/usr/bin/awk -f

# Convert column 1 from microseconds to seconds and strip comments
$0 !~ /^#/ {
  printf "%d.%06d", $1/1000000, $1 % 1000000
  $1=""
  print $0
}
