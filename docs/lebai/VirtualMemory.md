# 本文是在编译的时候要添加虚拟内存
具体如下
```bash
# 禁用交换空间
sudo swapoff -a
sudo fallocate -l 10G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h
```