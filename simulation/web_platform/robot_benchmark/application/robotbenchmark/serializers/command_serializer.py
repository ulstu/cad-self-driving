from rest_framework import serializers

from robotbenchmark.models import CommandQueue


class CommandQueueSerializer(serializers.ModelSerializer):
    """Сериализатор для команд"""

    class Meta:
        model = CommandQueue
        fields = '__all__'
