alias docker_clean_images='docker rmi $(docker images -a --filter=dangling=true -q)'
alias docker_clean_ps='docker rm $(docker ps --filter=status=exited --filter=status=created -q)'

docker kill $(docker ps -q)
docker_clean_ps
docker rmi $(docker images -a -q)