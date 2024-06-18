from rest_framework.response import Response
from rest_framework.views import APIView

from robotbenchmark.models import CommandQueue
from robotbenchmark.serializers.command_serializer import CommandQueueSerializer


class CommandQueueView(APIView):
    """Вью для команд хостовой машины"""

    def get(self, request, format=None):
        """Исполняет первую команду в очереди и удаляет ее из очереди"""
        command = CommandQueue.objects.first()
        if not command:
            return Response([], status=200)
        serializer = CommandQueueSerializer(command)
        command.delete()
        return Response(serializer.data)
