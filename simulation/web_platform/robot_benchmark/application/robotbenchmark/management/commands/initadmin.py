from django.contrib.auth.models import User
from django.core.management import BaseCommand


class Command(BaseCommand):
    help = "init admin in database"

    def handle(self, *args, **options):
        if not User.objects.filter(username="admin").exists():
            admin = User.objects.create_superuser(username="admin", password="admin")
            admin.is_active = True
            admin.is_admin = True
            admin.save()
        print("ADMIN SUCCESS CREATED")
