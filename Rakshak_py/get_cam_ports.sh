#!/usr/bin/env bash

shopt -s nullglob
declare -A VIDEO_PATHS
declare -A ACM_PATHS

get_serial() {
    udevadm info --name="$1" --query=property \
        | awk -F= '/^ID_SERIAL_SHORT=/ {print $2; exit}'
}

# Collect video devices
for vdev in /dev/video*; do
    serial=$(get_serial "$vdev")
    if [[ -n $serial && -z ${VIDEO_PATHS[$serial]} ]]; then
        VIDEO_PATHS["$serial"]="$vdev"
    fi
done

# Collect ACM devices
for tdev in /dev/ttyACM*; do
    serial=$(get_serial "$tdev")
    if [[ -n $serial && -z ${ACM_PATHS[$serial]} ]]; then
        ACM_PATHS["$serial"]="$tdev"
    fi
done



echo '{'
first=1
for serial in "${!VIDEO_PATHS[@]}" "${!ACM_PATHS[@]}"; do
    [[ $printed =~ (^| )$serial($| ) ]] && continue
    printed+="$serial "
    [[ $first -eq 0 ]] && echo ',' 
    first=0

    video="${VIDEO_PATHS[$serial]}"
    acm="${ACM_PATHS[$serial]}"

    printf '  "%s": {' "$serial" 

    if [[ -n $video && -n $acm ]]; then
        printf '"video": "%s", "serial": "%s"' "$video" "$acm" 
    elif [[ -n $video ]]; then
        printf '"video": "%s"' "$video" 
    elif [[ -n $acm ]]; then
        printf '"serial": "%s"' "$acm" 
    fi
    echo '}' 
done
echo '}' 

# Construct raw JSON using jq format
# echo '{' > out.json
# first=1
# for serial in "${!VIDEO_PATHS[@]}" "${!ACM_PATHS[@]}"; do
#     [[ $printed =~ (^| )$serial($| ) ]] && continue
#     printed+="$serial "
#     [[ $first -eq 0 ]] && echo ',' 
#     first=0

#     video="${VIDEO_PATHS[$serial]}"
#     acm="${ACM_PATHS[$serial]}"

#     printf '  "%s": {' "$serial" 

#     if [[ -n $video && -n $acm ]]; then
#         printf '"video": "%s", "serial": "%s"' "$video" "$acm" 
#     elif [[ -n $video ]]; then
#         printf '"video": "%s"' "$video" 
#     elif [[ -n $acm ]]; then
#         printf '"serial": "%s"' "$acm" 
#     fi
#     echo '}' 
# done
# echo '}' 