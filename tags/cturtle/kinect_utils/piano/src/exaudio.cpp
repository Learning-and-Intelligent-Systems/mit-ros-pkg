
////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include <SFML/Audio.hpp>
#include <iomanip>
#include <iostream>
#include <stdio.h>

#include "notespath.h"
////////////////////////////////////////////////////////////
/// Entry point of application
///
/// \return Application exit code
///
////////////////////////////////////////////////////////////
int main()
{
    // Load a sound buffer from a wav file
    sf::SoundBuffer Buffer;
    char tempname[500];
    sprintf(tempname,"%s/B.wav",notes_path);
    if (!Buffer.LoadFromFile(tempname))
        return EXIT_FAILURE;

    // Display sound informations
    std::cout << "sound.wav :" << std::endl;
    std::cout << " " << Buffer.GetDuration()      << " sec"           << std::endl;
    std::cout << " " << Buffer.GetSampleRate()    << " samples / sec" << std::endl;
    std::cout << " " << Buffer.GetChannelsCount() << " channels"      << std::endl;

    // Create a sound instance and play it
    sf::Sound Sound(Buffer);
    Sound.Play();
        sf::Sleep(0.4f);
    Sound.Play();
    sf::Sleep(0.4f);
Sound.Play();

    // Loop while the sound is playing
    while (Sound.GetStatus() == sf::Sound::Playing)
    {
        // Display the playing position
        std::cout << "\rPlaying... " << std::fixed << std::setprecision(2) << Sound.GetPlayingOffset() << " sec";

        // Leave some CPU time for other threads
        sf::Sleep(0.1f);
    }
    std::cout << std::endl;

    // Wait until the user presses 'enter' key
    std::cout << "Press enter to exit..." << std::endl;
    std::cin.ignore(10000, '\n');

    return EXIT_SUCCESS;
}
