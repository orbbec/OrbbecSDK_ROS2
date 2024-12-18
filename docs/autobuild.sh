#!/bin/bash

#set -e

##################################################################

##################################################################
#Function List
print_help()
{
    echo ""
    echo "------------------------ help menu --------------------------"
    echo " help menu option: "
    echo "./autobuild.sh all   ------build all"
    echo "./autobuild.sh senddoc  ----- send doc to server"
    echo "./autobuild.sh syncdoc  -----syncdoc"
    echo "./autobuild.sh clean   ----clean all"
    echo "./autobuild.sh run    ---- open html"
    echo "./autobuild.sh help  (-h, --help) show help menu"
    echo "-------------------------------------------------------------"
}

echo_autobuild_start()
{
    echo ""
    echo -e "\e[32m***********  ./autobuild.sh  $1 **************\e[0m"
}

build_all()
{
    echo "make clean && make html"
    make clean
    rm -rf _html
    rm -rf _build
    make html
	cp -R _build/html _html
    cp .nojekyll _html/
    echo "create _html finished"
}

clean()
{
    echo "clean all"
    make clean
}

sync_doc_old()
{
    echo "copy ../robot_sdk/ ../robot_ref/ ../robot_actuator file to robot_doc/source/syncdoc/"
    echo "search README.md"
    readme_files=$(find ../robot_sdk/ ../robot_ref/ ../robot_actuator/ -name 'README.md')
    for file in $readme_files; do
        dir=$(dirname "$file")

        new_dir=$(echo "$dir" | cut -d'/' -f2-)
        #echo "copy $file to source/syncdoc/$new_dir/README.md"
        echo "source/syncdoc/$new_dir/README.md"
        mkdir -p "source/syncdoc/$new_dir"
        cp -r "$file" "source/syncdoc/$new_dir/"
        if [ -d "$dir/image" ]; then
            cp -r "$dir/image" "source/syncdoc/$new_dir/"
        fi
    done
}
sync_doc()
{
    echo "search *.rst and *.md from source code"
    # readme_files=$(find ../robot_sdk/ ../robot_ref/ ../robot_actuator/ -name 'README.rst')
    # readme_files=$(find ../robot_sdk/ ../robot_ref/ ../robot_actuator/ -name '*.rst' -o -name '*.md')
    readme_files=$(find ../robot_sdk/ ../robot_ref/ ../robot_actuator/ -name 'README.rst' -o -name 'README.md')
    for file in $readme_files; do
        dir=$(dirname "$file")
        new_dir=$(echo "$dir" | cut -d'/' -f2-)
        #echo "copy $file to source/syncdoc/$new_dir/README.md"
        mkdir -p "source/syncdoc/$new_dir"
        cp -r "$file" "source/syncdoc/$new_dir/"
        echo "copy $file --> source/syncdoc/$new_dir/README.rst"
        echo "search image in $dir"
        find "$dir" -type f \( -name "*.png" -o -name "*.gif" -o -name "*.jpg" \) | while read -r serach_file; do
        target_path="source/syncdoc/$new_dir/$(echo "$serach_file" | sed "s|^$dir/||")"
        mkdir -p "$(dirname "$target_path")"
        cp "$serach_file" "$target_path"
        echo "copy $serach_file --> $target_path"
        done
    done
}

#####################################################################
# 初始化
#####################################################################

datetime=$(date +%Y%m%d_%H%M)
echo_autobuild_start;

##################################################################
if [ "$#" -eq "0" ]; then
    echo "unknown option:  $1 [$#]. pls enter correct cmd:"
	print_help;
	exit 0
fi

while [ $# -gt 0 ]; do
  echo "option=$1 total=$#:"
  case $1 in

    -h | --help | help | h | --h)
        print_help
        ;;

    clear | clean | cl)
     	clean
        ;;

    build | all)
        build_all
        ;;

    syncdoc | sync | sd)
	    sync_doc
        ;;

    run | r)
        echo "open html"
        # Google Chrome的完整路径
        CHROME_PATH="/usr/bin/google-chrome"
        # get full path name of html.html
        HTML_FILE="$(pwd)/_build/html/index.html"
        # 使用绝对路径确保脚本在任何目录下都能正确执行
        CHROME_EXECUTABLE="$(which "$CHROME_PATH")"
        # 使用绝对路径确保脚本在任何目录下都能正确执行
        CHROME_EXECUTABLE="$(which "$CHROME_PATH")"
        # x-www-browser _build/html/index
        # open -a /Applications/Firefox.app  _build/html/index
        "$CHROME_EXECUTABLE" "$HTML_FILE"
        ;;
    senddoc | send)
        echo "send doc update to 20.13"
        # echo "send doc update to 10.8.170.85"
        # sshpass -p pwd@123 scp -r _html/* orbbec@10.8.170.85:/var/www/html/robot_doc/
        OUT_IMAGE_SERVER_FOLDER=orbbec_sdk_ros2_doc
        sshpass -p 123 ssh guojia@10.8.180.197 "[ -d /home/orbbec/.SDKBuild/20_13_build_sdk/${OUT_IMAGE_SERVER_FOLDER} ] && echo 20.13 server ready || mkdir -p /home/orbbec/.SDKBuild/20_13_build_sdk/${OUT_IMAGE_SERVER_FOLDER}"
        sshpass -p 123 scp -r _html guojia@10.8.180.197:/home/orbbec/.SDKBuild/20_13_build_sdk/orbbec_sdk_ros2_doc/

        ;;
    test1 )
        echo "test1"

        ;;
    test2 )
        echo "test2"

        ;;
    test3 )
        echo "test3"

        ;;
    *)
        echo "unknown option:  $1"
        print_help
        ;;
    esac
    shift
done



echo "autobuild.sh end"

##################################################################

