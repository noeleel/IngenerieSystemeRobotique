

Pour réaliser des modifications sous Git, merci de suivre cette procédure:

Vous devez avoir quatre branches en tout: Sur le repo principal: Origin master -> noeleel/master Origin dev -> noeleel/dev Local master -> username/master Local dev -> username/dev

L'Origin master et le local master doivent toujours être à jour l'un sur l'autre. Vous ne devez réaliser vos modifications sur votre local dev!

Une fois que vos modifications sont fonctionnelles et testées, vous avez la possibilité de réaliser un merge entre votre local dev et votre local master, puis de pusher ces modiciations sur l'origin master.

Procédure classique : Après après réalisé le commit de vos modifications, faire un git pull puis un git rebase pour s'assurer que vous êtes à jour sur le git. Si conflit il y a lors du rebase, résoudre les conflits puis faire un continue rebase. Une fois que cette étape est réalisée, vous pouvez push vos commits en n'oubliant de mettre un message de la forme: "Tâche X : Permet d'afficher des couleurs (Description de la feature).

Procédure optionnelle: Si vous ne souhaitez pas que vos commits s'ajoutent tout en haut de l'arborescence des commits et que vous souhaitez simplement faire une modification, vous pouvez faire une branche temporaire qui sera à jour sur la branche dev et réaliser vos modifications sur cette branche dev2, les commit, puis merger cette branche avec votre branche dev avant de supprimer la branche dev2. Vous pouvez alors suivre la procédure classique.
