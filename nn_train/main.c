#define FANN_NO_DLL
#include "fann.h"
#include "fann_train.h"
#include <stdlib.h>

int main(int argc, char **argv)
{
	// 0 = production, 1 = recognition
	int nHidden = 50;
	int prevEpochs = 0;
	int nEpochs = 100;
	int mode = 0;
	struct fann *net;
	char *trainfile, *netfile, *prevfile;
	struct fann_train_data *data;

	if(mode)
	{
		trainfile = "recognition.train";
		netfile = (char*)malloc(256);
		sprintf(netfile, "recognition.net%d.%d", nEpochs, nHidden);
		prevfile = (char*)malloc(256);
		sprintf(prevfile, "recognition.net%d.%d", prevEpochs, nHidden);
		if(!(net = fann_create_from_file(prevfile)))
			net = fann_create_standard(3, 2520, nHidden, 5);
		else
			nEpochs -= prevEpochs;
	}
	else
	{
		trainfile = "production.train";
		netfile = (char*)malloc(256);
		sprintf(netfile, "production.net%d.%d", nEpochs, nHidden);
		prevfile = (char*)malloc(256);
		sprintf(prevfile, "production.net%d.%d", prevEpochs, nHidden);
		if(!(net = fann_create_from_file(prevfile)))
			net = fann_create_standard(3, 131, nHidden, 2394);
		else
			nEpochs -= prevEpochs;
	}

	data = fann_read_train_from_file(trainfile);
	
	fann_set_activation_function_hidden(net, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(net, FANN_SIGMOID_SYMMETRIC);
	fann_set_training_algorithm(net, FANN_TRAIN_RPROP);
	//fann_set_input_scaling_params(net, data, -1.0, 1.0);
	//fann_set_output_scaling_params(net, data, -1.0, 1.0);

	printf("Starting training on %s (%d, %d)\n", trainfile, nEpochs, nHidden);
	fann_train_on_data(net, data, nEpochs, 1, 0.0001);
	printf("Training complete\n");

	fann_save(net, netfile);
	fann_destroy(net);
	return 0;
}