from django.contrib import admin
from .models import Problem, TournamentUser, Tournament, ProblemUser, CommandQueue


# Register your models here.


@admin.register(Problem)
class ProblemAdmin(admin.ModelAdmin):
    list_display = ('id', 'title', 'world_path', 'difficulty', 'image', 'author')  # Список полей, которые будут отображаться в списке объектов
    list_display_links = ('id', 'title', 'world_path', 'difficulty')  # Список полей, которые будут отображаться в списке объектов
    list_filter = ('id', 'title', 'author')  # Список полей, по которым можно будет фильтровать объекты
    search_fields = ('id', 'title', 'author')  # Список полей, по которым можно будет искать объекты
    ordering = ('id',)  # Сортировка объектов по умолчанию


@admin.register(ProblemUser)
class ProblemUserAdmin(admin.ModelAdmin):
    list_display = ('id', 'user', 'problem', 'is_completed', 'points')  # Список полей, которые будут отображаться в списке объектов
    list_display_links = ('id', 'problem', 'is_completed', 'points')  # Список полей, которые будут отображаться в списке объектов
    list_filter = ('id', 'problem', 'points')  # Список полей, по которым можно будет фильтровать объекты
    search_fields = ('id', 'problem', 'points')  # Список полей, по которым можно будет искать объекты
    ordering = ('id',)  # Сортировка объектов по умолчанию


@admin.register(Tournament)
class TournamentAdmin(admin.ModelAdmin):
    list_display = ('id', 'name', 'date_start', 'date_end')  # Список полей, которые будут отображаться в списке объектов
    list_display_links = ('id', 'name')  # Список полей, которые будут отображаться в списке объектов
    list_filter = ('id', 'name', 'date_start', 'date_end')  # Список полей, по которым можно будет фильтровать объекты
    search_fields = ('id', 'name', 'date_start')  # Список полей, по которым можно будет искать объекты
    ordering = ('id',)  # Сортировка объектов по умолчанию


@admin.register(TournamentUser)
class TournamentUserAdmin(admin.ModelAdmin):
    list_display = ('id', 'user', 'tournament', 'is_completed', 'points')  # Список полей, которые будут отображаться в списке объектов
    list_display_links = ('id', 'tournament', 'is_completed', 'points')  # Список полей, которые будут отображаться в списке объектов
    list_filter = ('id', 'user', 'tournament', 'is_completed', 'points') # Список полей, по которым можно будет фильтровать объекты
    search_fields = ('id', 'user', 'tournament')  # Список полей, по которым можно будет искать объекты
    ordering = ('id',)  # Сортировка объектов по умолчанию


@admin.register(CommandQueue)
class CommandQueueAdmin(admin.ModelAdmin):
    list_display = ('id', 'command')  # Список полей, которые будут отображаться в списке объектов
    list_display_links = ('id', 'command')  # Список полей, которые будут отображаться в списке объектов
    search_fields = ('id', 'command')  # Список полей, по которым можно будет искать объекты
    ordering = ('id',)  # Сортировка объектов по умолчанию
