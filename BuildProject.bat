xmake
if exist bin\windows_x64_debug\ (
	md bin\windows_x64_debug\assets
	xcopy /s/y bin\assets bin\windows_x64_debug\assets
)

if exist bin\windows_x64_release\ (
	md bin\windows_x64_release\assets
	xcopy /s/y bin\assets bin\windows_x64_release\assets
)


pause