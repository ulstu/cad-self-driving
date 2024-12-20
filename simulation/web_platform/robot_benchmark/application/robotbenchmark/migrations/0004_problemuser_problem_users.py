# Generated by Django 5.0.2 on 2024-02-23 09:19

import django.db.models.deletion
from django.conf import settings
from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('robotbenchmark', '0003_problem_image_path'),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name='ProblemUser',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('is_completed', models.BooleanField(default=False)),
                ('points', models.IntegerField(default=0)),
                ('problem', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, to='robotbenchmark.problem')),
                ('user', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, to=settings.AUTH_USER_MODEL)),
            ],
        ),
        migrations.AddField(
            model_name='problem',
            name='users',
            field=models.ManyToManyField(related_name='user_problems', through='robotbenchmark.ProblemUser', to=settings.AUTH_USER_MODEL),
        ),
    ]
