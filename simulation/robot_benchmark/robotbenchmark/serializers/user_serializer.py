from rest_framework import serializers
from django.contrib.auth import get_user_model

UserModel = get_user_model()


class UserSerializer(serializers.ModelSerializer):
    class Meta:
        model = UserModel
        #fields = ["url", "username", "password"]
        exclude = ['last_login', 'is_superuser', 'is_staff', 'groups', 'user_permissions', 'date_joined', 'is_active']
        extra_kwargs = {"password": {"write_only": True}}

    def create(self, validated_data):
        user = UserModel(username=validated_data["username"])
        user.set_password(validated_data["password"])
        user.save()
        return user

