from django.contrib.auth.password_validation import validate_password
from rest_framework import serializers
from django.contrib.auth import get_user_model

UserModel = get_user_model()


class UserSerializer(serializers.ModelSerializer):
    class Meta:
        model = UserModel
        exclude = ['last_login', 'is_staff', 'groups', 'user_permissions', 'date_joined', 'is_active']
        extra_kwargs = {
            "password": {"write_only": True}
        }

    def create(self, validated_data):
        user = UserModel(
            username=validated_data["username"],
            first_name=validated_data['first_name'],
            last_name=validated_data['last_name'],
            email=validated_data['email'],
            is_superuser=validated_data['is_superuser'],
        )
        user.set_password(validated_data["password"])
        user.save()
        return user

    def validate_password(self, value: str) -> str:
        validate_password(value)
        return value
