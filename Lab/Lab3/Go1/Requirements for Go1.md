for Go1 DogInstall Micromamba

# make a bin folder (common convention)
mkdir -p ~/bin

# download + extract micromamba binary into ~/bin
curl -L https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xvj -C ~/bin --strip-components=1 bin/micromamba

# add ~/bin to PATH for future terminals
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc

# verify
micromamba --version



micromamba shell init -s bash


exec bash


# create
micromamba create -n py38 -y python=3.8 pip
# activate
micromamba activate py38
# check python version
python -V
