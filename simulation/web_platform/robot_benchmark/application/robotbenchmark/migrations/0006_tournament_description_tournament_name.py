# Generated by Django 5.0.2 on 2024-04-13 12:20

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('robotbenchmark', '0005_tournament_tournamentuser_tournament_users'),
    ]

    operations = [
        migrations.AddField(
            model_name='tournament',
            name='description',
            field=models.CharField(default=None, max_length=5000),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='tournament',
            name='name',
            field=models.CharField(default=None, max_length=255),
            preserve_default=False,
        ),
    ]
