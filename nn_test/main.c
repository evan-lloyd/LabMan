#define FANN_NO_DLL
#include "fann.h"
#include "fann_train.h"
#include <stdlib.h>

typedef struct fann_train_data traindata;

int main(int argc, char **argv)
{
	// 0 = production, 1 = recognition
	int mode = argv[1][27] == 'r';
	struct fann *net;
	char *testfile, *netfile = argv[1];
	traindata *data;

	if(mode)
	{
		testfile = "recognition.test";
		//netfile = "recognition.net";
	}
	else
	{
		testfile = "production.test";
		//netfile = "production.net";
	}

	net = fann_create_from_file(netfile);
	
	fann_set_activation_function_hidden(net, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(net, FANN_SIGMOID_SYMMETRIC);
	fann_set_training_algorithm(net, FANN_TRAIN_RPROP);

	printf("Starting testing on %s\n", testfile);
	data = fann_read_train_from_file(testfile);
	fann_test_data(net, data);
	printf("Testing complete: MSE = %f\n", fann_get_MSE(net));

	fann_save(net, netfile);
	fann_destroy(net);
	return 0;
}