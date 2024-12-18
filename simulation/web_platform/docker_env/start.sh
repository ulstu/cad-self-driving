make all FLAVOR="devel" ROBOT_PANEL_PORT=8008 VS_PORT=31415 WEBOTS_STREAM_PORT=1234
docker exec -it ulstu-devel sudo sed -i 's/\r$//g' /ulstu/.bashrc

