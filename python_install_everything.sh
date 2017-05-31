# (re)install all the library the robot needs
# make sure everything is in this file!

repo_list="libmotors libAX12 robot-framework GrobotControl"
force_master=false
pull_enable=true

for repo in $repo_list
do
  echo $repo
  cd /home/pi/$repo

  if [ "$force_master" = true ] ; then
    git checkout master
  fi
  if [ "$pull_enable" = true ] ; then
    git pull
  fi

  make clean
  make
  make install
done

cd /home/pi/GrobotControl
