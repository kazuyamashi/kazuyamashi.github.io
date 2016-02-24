#ROSパッケージリリースの仕方
このセクションでは独自のROSパッケージを公式wikiへの公開方法を学びます。  
ROSのパッケージは基本的にGithubで管理するのが一般的です。  
Githubのアカウントを持っていない場合はアカウントの作成をしましょう。

まず，Githubのリポジトリ[ros/rosdistro](https://github.com/ros/rosdistro/)をフォークします。  
以下のURLでリポジトリへアクセスします。  
https://github.com/ros/rosdistro/  

<img src="pic/rosdistro.png" height=500>

ros/rosdistroのフォークが終了したら，任意の場所へgit colneします。

```
git clone https://github.com/your_GithubID/rosdistro.git
```
パッケージのリリースのためには`rosditro/indigo/distribution.yaml`を編集します。  
ファイル内にはアルファベット順でパッケージが定義されています。編集する際はアルファベット順を崩さないように注意してください。  

ファイル編集を行う際の各パッケージの書式は以下のようなものです。  
`your_package_name`には開発したパッケージ名とします。  

```yaml
  your_package_name:
    doc:
      type: git
      url: https://github.com/your_GithubID/repositry_name.git
      version: master
    source:
      type: git
      url: https://github.com/your_GithubID/repositry_name.git
      version: master
```

ファイルの編集が終了したら，rosdistro内でpythonコマンドnosetestsを実行します。

```
cd rosdistro
rosdistro_reformat index.yaml
nosetests
```

nosetestsが成功したら以下のような出力がされます。

```
Ran 6 tests in 45.464s

OK
```

nosetestsが終了したら変更点を更新します。  

```
git add .
git commit -m "update indigo/distribution.yaml"
git push
```

フォークしたリポジトリのURLに再度アクセスし，`New pull request`を行います。  

<img src="pic/rosdistro_user.png" height=500>

pull requestを行うとしばらくしてから，ROSのメンテナによってマージされます。  
また1日ほどでROSの公式Wikiにも掲載されます。  

<img src="pic/released.png" width=700>