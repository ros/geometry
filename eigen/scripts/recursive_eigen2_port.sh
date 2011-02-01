for f in $(rospack depends-on1 eigen); do
    echo "Converting $f to Eigen2 support "
    rosrun eigen e2_to_eigen2support.sh $(rospack find $f)
done